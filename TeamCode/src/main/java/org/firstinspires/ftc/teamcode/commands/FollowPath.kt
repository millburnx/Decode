package org.firstinspires.ftc.teamcode.commands

import com.millburnx.cmdx.Command
import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.util.WaitFor

fun FollowPathCommand(follower: Follower, path: PathChain, name: String = "Follow Pedro Path"): Command {
    return Command(name) {
        FollowPath(follower, path)
    }
}

suspend fun Command.FollowPath(follower: Follower, path: PathChain) {
    follower.followPath(path)
    WaitFor { !follower.isBusy }
}