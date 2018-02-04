/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Robert Haschke */

#include <moveit/task_constructor/container_p.h>

#include <moveit/task_constructor/introspection.h>
#include <ros/console.h>

#include <memory>
#include <iostream>
#include <algorithm>
#include <boost/range/adaptor/reversed.hpp>
#include <functional>

using namespace std::placeholders;

namespace moveit { namespace task_constructor {

ContainerBasePrivate::const_iterator ContainerBasePrivate::position(int index) const {
	const_iterator position = children_.begin();
	if (index > 0) {
		for (auto end = children_.end(); index > 0 && position != end; --index)
			++position;
	} else if (++index <= 0) {
		container_type::const_reverse_iterator from_end = children_.rbegin();
		for (auto end = children_.rend(); index < 0 && from_end != end; ++index)
			++from_end;
		position = from_end.base();
	}
	return position;
}

bool ContainerBasePrivate::traverseStages(const ContainerBase::StageCallback &processor,
                                          unsigned int cur_depth, unsigned int max_depth) const {
	if (cur_depth >= max_depth)
		return true;

	for (auto &stage : children_) {
		if (!processor(*stage, cur_depth))
			continue;
		const ContainerBasePrivate *container = dynamic_cast<const ContainerBasePrivate*>(stage->pimpl());
		if (container)
			container->traverseStages(processor, cur_depth+1, max_depth);
	}
	return true;
}

bool ContainerBasePrivate::canCompute() const
{
	// call the method of the public interface
	return static_cast<ContainerBase*>(me_)->canCompute();
}

bool ContainerBasePrivate::compute()
{
	// call the method of the public interface
	return static_cast<ContainerBase*>(me_)->compute();
}

void ContainerBasePrivate::copyState(Interface::iterator external,
                                     Stage &child, bool to_start, bool updated) {
	if (to_start) {
		InterfaceState& internal = *child.pimpl()->starts()->clone(*external);
		internal_to_external_.insert(std::make_pair(&internal, external));
	} else {
		InterfaceState& internal = *child.pimpl()->ends()->clone(*external);
		internal_to_external_.insert(std::make_pair(&internal, external));
	}
}

ContainerBase::ContainerBase(ContainerBasePrivate *impl)
   : Stage(impl)
{
}

size_t ContainerBase::numChildren() const
{
	return pimpl()->children().size();
}

bool ContainerBase::traverseChildren(const ContainerBase::StageCallback &processor) const
{
	return pimpl()->traverseStages(processor, 0, 1);
}
bool ContainerBase::traverseRecursively(const ContainerBase::StageCallback &processor) const
{
	if (!processor(*this, 0))
		return false;
	return pimpl()->traverseStages(processor, 1, UINT_MAX);
}

bool ContainerBase::insert(Stage::pointer &&stage, int before)
{
	StagePrivate *impl = stage->pimpl();
	if (impl->parent() != nullptr || numSolutions() != 0) {
		ROS_ERROR("cannot re-parent stage");
		return false;
	}

	ContainerBasePrivate::const_iterator where = pimpl()->position(before);
	ContainerBasePrivate::iterator it = pimpl()->children_.insert(where, std::move(stage));
	impl->setHierarchy(this, it);
	return true;
}

bool ContainerBase::remove(int pos)
{
	ContainerBasePrivate::const_iterator it = pimpl()->position(pos);
	pimpl()->children_.erase(it);
	return true;
}

void ContainerBase::clear()
{
	pimpl()->children_.clear();
}

void ContainerBase::reset()
{
	auto impl = pimpl();

	// recursively reset children
	for (auto& child: impl->children())
		child->reset();

	// clear mapping
	impl->internal_to_external_.clear();

	Stage::reset();
}

void ContainerBase::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	InitStageException errors;
	auto impl = pimpl();
	auto& children = impl->children();

	Stage::init(scene);

	// we need to have some children to do the actual work
	if (children.empty()) {
		errors.push_back(*this, "no children");
		throw errors;
	}

	// recursively init all children
	for (auto& child : children) {
		try {
			child->init(scene);
		} catch (InitStageException &e) {
			errors.append(e);
		}
	}

	if (errors)
		throw errors;
}


SerialContainerPrivate::SerialContainerPrivate(SerialContainer *me, const std::string &name)
   : ContainerBasePrivate(me, name)
{
	// these lists don't need a notify function, connections are handled by onNewSolution()
	pending_backward_.reset(new Interface(Interface::NotifyFunction()));
	pending_forward_.reset(new Interface(Interface::NotifyFunction()));
}


struct SolutionCollector {
	SolutionCollector(size_t max_depth) : max_depth(max_depth) {}

	void operator()(const SerialContainer::solution_container& trace, double cost) {
		// traced path should not extend past container boundaries
		assert(trace.size() <= max_depth);

		if (trace.size() == max_depth) // reached max depth
			solutions.emplace_back(std::make_pair(trace, cost));
	}

	std::list<std::pair<SerialContainer::solution_container, double>> solutions;
	const size_t max_depth;
};

void SerialContainer::onNewSolution(const SolutionBase &current)
{
	auto impl = pimpl();
	const StagePrivate *creator = current.creator();
	auto& children = impl->children();

	// find number of stages before and after creator stage
	size_t num_before = 0, num_after = 0;
	for (auto it = children.begin(), end = children.end(); it != end; ++it, ++num_before)
		if ((*it)->pimpl() == creator)
			break;
	assert(num_before < children.size());  // creator should be one of our children
	num_after = children.size()-1 - num_before;

	SerialContainer::solution_container trace; trace.reserve(children.size());

	// find all incoming solution pathes ending at current solution
	SolutionCollector incoming(num_before);
	traverse<BACKWARD>(current, std::ref(incoming), trace);

	// find all outgoing solution pathes starting at current solution
	SolutionCollector outgoing(num_after);
	traverse<FORWARD>(current, std::ref(outgoing), trace);

	// collect (and sort) all solutions spanning from start to end of this container
	ordered<SerialSolution> sorted;
	SerialContainer::solution_container solution;
	solution.reserve(children.size());
	for (auto& in : incoming.solutions) {
		for (auto& out : outgoing.solutions) {
			InterfaceState::Priority prio(in.first.size() + 1 + out.first.size(),
			                              in.second + current.cost() + out.second);
			// found a complete solution path connecting start to end?
			if (prio.depth() == children.size()) {
				assert(solution.empty());
				// insert incoming solutions in reverse order
				solution.insert(solution.end(), in.first.rbegin(), in.first.rend());
				// insert current solution
				solution.push_back(&current);
				// insert outgoing solutions in normal order
				solution.insert(solution.end(), out.first.begin(), out.first.end());
				// store solution in sorted list
				sorted.insert(SerialSolution(impl, std::move(solution), prio.cost()));
			} else {
				// update state costs
				const InterfaceState* start = (in.first.empty() ? current : *in.first.back()).start();
				start->owner()->updatePriority(*const_cast<InterfaceState*>(start), prio);
				const InterfaceState* end = (out.first.empty() ? current : *out.first.back()).end();
				end->owner()->updatePriority(*const_cast<InterfaceState*>(end), prio);
			}
		}
	}

	// store new solutions (in sorted)
	for (auto it = sorted.begin(), end = sorted.end(); it != end; ++it)
		impl->storeNewSolution(std::move(*it));
}

void SerialContainerPrivate::storeNewSolution(SerialSolution &&s)
{
	const InterfaceState *internal_from = s.internalStart();
	const InterfaceState *internal_to = s.internalEnd();

	// create new SerialSolution and get a reference to it
	SerialSolution& solution = *solutions_.insert(std::move(s));

	// add solution to existing or new start state
	auto it = internal_to_external_.find(internal_from);
	if (it != internal_to_external_.end()) {
		// connect solution to existing start state
		solution.setStartState(*it->second);
	} else {
		// spawn a new state in previous stage
		Interface::iterator external = prevEnds()->add(InterfaceState(*internal_from), NULL, &solution);
		internal_to_external_.insert(std::make_pair(internal_from, external));
	}

	// add solution to existing or new end state
	it = internal_to_external_.find(internal_to);
	if (it != internal_to_external_.end()) {
		// connect solution to existing start state
		solution.setEndState(*it->second);
	} else {
		// spawn a new state in next stage
		Interface::iterator external = nextStarts()->add(InterfaceState(*internal_to), &solution, NULL);
		internal_to_external_.insert(std::make_pair(internal_to, external));
	}

	// perform default stage action on new solution
	newSolution(solution);
}


SerialContainer::SerialContainer(SerialContainerPrivate *impl)
   : ContainerBase(impl)
{}
SerialContainer::SerialContainer(const std::string &name)
   : SerialContainer(new SerialContainerPrivate(this, name))
{}

void SerialContainer::reset()
{
	auto impl = pimpl();

	// clear queues
	impl->solutions_.clear();
	impl->pending_backward_->clear();
	impl->pending_forward_->clear();

	// recursively reset children
	ContainerBase::reset();
}

void SerialContainerPrivate::connect(StagePrivate* prev, StagePrivate* next) {
	prev->setNextStarts(next->starts());
	next->setPrevEnds(prev->ends());
}

void SerialContainer::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	InitStageException errors;
	auto impl = pimpl();

	// if there are no children, there is nothing to connect
	if (!impl->children().empty()) {
		/*** connect children ***/
		// first stage sends backward to pending_backward_
		auto start = impl->children().begin();
		(*start)->pimpl()->setPrevEnds(impl->pending_backward_);

		// last stage sends forward to pending_forward_
		auto last = --impl->children().end();
		(*last)->pimpl()->setNextStarts(impl->pending_forward_);

		auto cur = start;
		auto prev = cur; ++cur; // prev points to 1st, cur points to 2nd stage
		if (prev != last) {// we have more than one children
			auto next = cur; ++next; // next points to 3rd stage (or end)
			for (; cur != last; ++prev, ++cur, ++next) {
				impl->connect(**prev, **cur);
				impl->connect(**cur, **next);
			}
			// finally connect last == cur and prev stage
			impl->connect(**prev, **cur);
		}

		// recursively init + validate all children
		// this needs to be done *after* initializing the connections
		ContainerBase::init(scene);

		// initialize starts_ and ends_ interfaces
		Stage* child = start->get();
		if (child->pimpl()->starts())
			impl->starts_.reset(new Interface([impl, child](Interface::iterator external, bool updated){
				// new external state in our starts_ interface is copied to first child
				impl->copyState(external, *child, true, updated);
			}));

		child = last->get();
		if (child->pimpl()->ends())
			impl->ends_.reset(new Interface([impl, child](Interface::iterator external, bool updated){
				// new external state in our ends_ interface is copied to last child
				impl->copyState(external, *child, false, updated);
			}));

		// validate connectivity of this
		if (!impl->nextStarts())
			errors.push_back(*this, "cannot sendForward()");
		if (!impl->prevEnds())
			errors.push_back(*this, "cannot sendBackward()");
	} else {
		errors.push_back(*this, "no children");
		// no children -> no reading
		impl->starts_.reset();
		impl->ends_.reset();
	}

	if (errors)
		throw errors;
}

bool SerialContainer::canCompute() const
{
	return !pimpl()->children().empty();
}

bool SerialContainer::compute()
{
	bool computed = false;
	for(const auto& stage : pimpl()->children()) {
		if(!stage->pimpl()->canCompute())
			continue;
		std::cout << "Computing stage '" << stage->name() << "':" << std::endl;
		bool success = stage->pimpl()->compute();
		computed = true;
		std::cout << (success ? "succeeded" : "failed") << std::endl;
	}
	return computed;
}

size_t SerialContainer::numSolutions() const
{
	return pimpl()->solutions_.size();
}

void SerialContainer::processSolutions(const ContainerBase::SolutionProcessor &processor) const
{
	for(const SolutionBase& s : pimpl()->solutions())
		if (!processor(s))
			break;
}

template <TraverseDirection dir>
void SerialContainer::traverse(const SolutionBase &start, const SolutionProcessor &cb,
                               solution_container &trace, double trace_cost)
{
	const InterfaceState::Solutions& solutions = trajectories<dir>(start);
	if (solutions.empty())  // if we reached the end, call the callback
		cb(trace, trace_cost);
	else for (SolutionBase* successor : solutions) {
		trace.push_back(successor);
		trace_cost += successor->cost();

		traverse<dir>(*successor, cb, trace, trace_cost);

		trace_cost -= successor->cost();
		trace.pop_back();
	}
}

void SerialSolution::fillMessage(moveit_task_constructor_msgs::Solution &msg,
                                 Introspection* introspection = nullptr) const
{
	moveit_task_constructor_msgs::SubSolution sub_msg;
	sub_msg.id = introspection ? introspection->solutionId(*this) : 0;
	sub_msg.cost = this->cost();

	const Introspection *ci = introspection;
	sub_msg.stage_id = ci ? ci->stageId(this->creator()->me()) : 0;

	sub_msg.sub_solution_id.reserve(subsolutions_.size());
	if (introspection) {
		for (const SolutionBase* s : subsolutions_)
			sub_msg.sub_solution_id.push_back(introspection->solutionId(*s));
		msg.sub_solution.push_back(sub_msg);
	}

	msg.sub_trajectory.reserve(msg.sub_trajectory.size() + subsolutions_.size());
	for (const SolutionBase* s : subsolutions_)
		s->fillMessage(msg, introspection);
}


ParallelContainerBasePrivate::ParallelContainerBasePrivate(ParallelContainerBase *me, const std::string &name)
   : ContainerBasePrivate(me, name)
{
	starts_.reset(new Interface(std::bind(&ParallelContainerBase::onNewStartState, me, _1, _2)));
	ends_.reset(new Interface(std::bind(&ParallelContainerBase::onNewEndState, me, _1, _2)));
}


ParallelContainerBase::ParallelContainerBase(ParallelContainerBasePrivate *impl)
   : ContainerBase(impl)
{}
ParallelContainerBase::ParallelContainerBase(const std::string &name)
   : ParallelContainerBase(new ParallelContainerBasePrivate(this, name))
{}

void ParallelContainerBase::reset()
{
	// recursively reset children
	ContainerBase::reset();
}

void ParallelContainerBase::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	InitStageException errors;
	auto impl = pimpl();

	// connect children such that they directly send to this' prevEnds() / nextStarts()
	for (const Stage::pointer& stage : impl->children()) {
		StagePrivate *child = stage->pimpl();
		child->setPrevEnds(impl->prevEnds());
		child->setNextStarts(impl->nextStarts());
	}

	// recursively init + validate all children
	// this needs to be done *after* initializing the connections
	ContainerBase::init(scene);

	if (errors)
		throw errors;
}

void ParallelContainerBase::onNewSolution(const SolutionBase &s)
{
	// update state priorities
	InterfaceState::Priority prio(1, s.cost());
	InterfaceState* start = const_cast<InterfaceState*>(s.start());
	start->owner()->updatePriority(*start, prio);
	InterfaceState* end = const_cast<InterfaceState*>(s.end());
	end->owner()->updatePriority(*end, prio);

	pimpl()->newSolution(s);
}


WrapperBasePrivate::WrapperBasePrivate(WrapperBase *me, const std::string &name)
   : ContainerBasePrivate(me, name)
{
	dummy_starts_.reset(new Interface(Interface::NotifyFunction()));
	dummy_ends_.reset(new Interface(Interface::NotifyFunction()));
}

WrapperBase::WrapperBase(const std::string &name, Stage::pointer &&child)
   : WrapperBase(new WrapperBasePrivate(this, name), std::move(child))
{}

WrapperBase::WrapperBase(WrapperBasePrivate *impl, Stage::pointer &&child)
   : ContainerBase(impl)
{
	if (child) insert(std::move(child));
}

bool WrapperBase::insert(Stage::pointer &&stage, int before)
{
	// restrict num of children to one
	if (numChildren() > 0)
		return false;
	return ContainerBase::insert(std::move(stage), before);
}

void WrapperBase::reset()
{
	pimpl()->dummy_starts_->clear();
	pimpl()->dummy_ends_->clear();
}

void WrapperBase::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	auto impl = pimpl();

	if (numChildren() != 1)
		throw InitStageException(*this, "no wrapped child");

	// as a generator-like stage, we don't accept inputs
	assert(!impl->starts());
	assert(!impl->ends());

	// provide a dummy interface to receive interface states of wrapped child
	wrapped()->pimpl()->setPrevEnds(impl->dummy_ends_);
	wrapped()->pimpl()->setNextStarts(impl->dummy_starts_);

	// init + validate children
	ContainerBase::init(scene);
}

size_t WrapperBase::numSolutions() const
{
	// dummy implementation needed to allow insert() in constructor
	return 0;
}

Stage* WrapperBase::wrapped()
{
	return pimpl()->children().empty() ? nullptr : pimpl()->children().front().get();
}

void WrapperBase::onNewSolution(const SolutionBase &s)
{
	// update state priorities
	InterfaceState::Priority prio(1, s.cost());
	InterfaceState* start = const_cast<InterfaceState*>(s.start());
	start->owner()->updatePriority(*start, prio);
	InterfaceState* end = const_cast<InterfaceState*>(s.end());
	end->owner()->updatePriority(*end, prio);

	pimpl()->newSolution(s);
}


WrapperPrivate::WrapperPrivate(Wrapper *me, const std::string &name)
   : WrapperBasePrivate(me, name)
{}

Wrapper::Wrapper(WrapperPrivate *impl, Stage::pointer &&child)
   : WrapperBase(impl, std::move(child))
{}
Wrapper::Wrapper(const std::string &name, Stage::pointer &&child)
   : Wrapper(new WrapperPrivate(this, name), std::move(child))
{}

void Wrapper::reset()
{
	WrapperBase::reset();
	pimpl()->solutions_.clear();
}

bool Wrapper::canCompute() const
{
	return wrapped()->pimpl()->canCompute();
}

bool Wrapper::compute()
{
	size_t num_before = numSolutions();
	wrapped()->pimpl()->compute();
	return numSolutions() > num_before;
}

size_t Wrapper::numSolutions() const
{
	return pimpl()->solutions_.size();
}

size_t Wrapper::numFailures() const
{
	return pimpl()->failures_.size();
}

void Wrapper::processSolutions(const Stage::SolutionProcessor &processor) const
{
	for(const auto& s : pimpl()->solutions_)
		if (!processor(*s))
			break;
}

void Wrapper::processFailures(const Stage::SolutionProcessor &processor) const
{
	for(const auto& s : pimpl()->failures_)
		if (!processor(*s))
			break;
}

void Wrapper::spawn(InterfaceState &&state, std::unique_ptr<SolutionBase>&& s)
{
	auto impl = pimpl();
	s->setCreator(impl);
	SolutionBase* solution = s.get();
	if (s->isFailure()) {
		impl->failure_states_.emplace_back(std::move(state));
		s->setStartState(impl->failure_states_.back());
		s->setEndState(impl->failure_states_.back());
		impl->failures_.emplace_back(std::move(s));
	} else {
		impl->solutions_.insert(std::move(s));
		impl->prevEnds()->add(InterfaceState(state), NULL, solution);
		impl->nextStarts()->add(std::move(state), solution, NULL);
	}
	impl->newSolution(*solution);
}

} }